"""Carry Q0 tool calls through authenticated Codex CLI structured output.

Q0 still owns tool execution, sandbox, history and plan validation. Codex's own
shell/apps/browser tools are disabled; there is no API-key fallback.
"""
import json,os,subprocess,time,uuid
from pathlib import Path
from ..harness.api import API,APIError,image_count
from ..runtime.io_utils import atomic

class CodexSubscriptionAPI(API):
    def __init__(self,root,config):
        if config.get('model')!='gpt-6-astra':raise ValueError('This integration requires the user-selected gpt-6-astra')
        super().__init__(root,{**config,'provider':'openai'})
        self.codex=config.get('codex_executable','codex')
        self.env=os.environ.copy()
        for name in ['OPENAI_API_KEY','CODEX_API_KEY','ANTHROPIC_AUTH_TOKEN','OPENAI_BASE_URL','OPENAI_API_BASE']:self.env.pop(name,None)
        auth=subprocess.run([self.codex,'login','status'],capture_output=True,text=True,env=self.env,timeout=20)
        if auth.returncode or 'ChatGPT' not in auth.stdout+auth.stderr:
            raise APIError('subscription_login_required','Codex must already be logged in with ChatGPT')

    def _call(self,payload,req,key_env):
        if image_count(payload):raise APIError("vision_disabled", "Subscription adapter currently accepts structured text only")
        self.index+=1
        folder=self.root/'api'/f'{self.index:05d}_{req["kind"]}_{req["observation_id"]}'/'attempt_0'
        folder.mkdir(parents=True)
        item={'type':'object','additionalProperties':False,'required':['name','arguments_json'],
              'properties':{'name':{'type':'string','enum':[t['name'] for t in payload['tools']]},'arguments_json':{'type':'string'}}}
        schema={'type':'object','additionalProperties':False,'required':['calls'],
                'properties':{'calls':{'type':'array','minItems':1,'maxItems':1,'items':item}}}

        atomic(folder/'output_schema.json',schema);atomic(folder/'request.json',payload)
        if req.get('context_selection') is not None:atomic(folder/'context_selection.json',req['context_selection'])
        prompt=('You are the model inside the Q0 robotics harness. Do not use Codex native tools. '
                'Return exactly one Q0 tool call through the response schema: name and JSON-encoded arguments_json. '
                'The external Q0 runner will execute it and return its result in the next request. '
                'Follow the supplied instructions and tool schemas. Never pretend a tool has run.\n'+json.dumps(payload,ensure_ascii=False))
        if len(prompt.encode())>self.context_max_bytes:raise APIError('api_context_size_limit')
        command=[self.codex,'exec','--ignore-user-config','--skip-git-repo-check','--ephemeral',
                 '--model',self.model,'--sandbox','read-only','--disable','shell_tool','--disable','apps',
                 '--disable','browser_use','--disable','computer_use','--json','--color','never',
                 '-c','model_reasoning_effort="'+self.config.get('codex_reasoning_effort','medium')+'"',
                 '--cd',str(folder),'--output-schema',str(folder/'output_schema.json'),
                 '--output-last-message',str(folder/'structured_reply.json'),'-']
        atomic(folder/'transport.json',dict(transport='codex_cli_chatgpt_subscription',model=self.model,native_tools_disabled=True,api_key_fallback=False))
        seconds=min(float(self.config.get('api_request_timeout_s',180)),req['deadline']-time.time()-1)
        if seconds<=0:raise APIError('api_request_deadline')
        (folder/'prompt.txt').write_text(prompt)
        until=time.time()+seconds
        self.check_alive(min(until,req['deadline']-1))
        with (folder/'prompt.txt').open() as stdin,(folder/'events.jsonl').open('w') as events,(folder/'transport.stderr').open('w') as stderr:
            child=subprocess.Popen(command,stdin=stdin,stdout=events,stderr=stderr,env=self.env)
            try:
                while child.poll() is None:
                    self.check_alive(min(until,req['deadline']-1))
                    time.sleep(.1)
                returncode=child.returncode
            finally:
                if child.poll() is None:child.kill();child.wait(timeout=10)
        if returncode:raise APIError('subscription_request_failed',f'Codex exit={returncode}; inspect {folder}/transport.stderr')
        events=[json.loads(x) for x in (folder/'events.jsonl').read_text().splitlines() if x.strip()]
        forbidden={'command_execution','mcp_tool_call','web_search','file_change'}
        if any(e.get('item',{}).get('type') in forbidden for e in events):raise APIError('unexpected_native_tool','No Q0 action executed')
        if not any(e.get('type')=='turn.completed' for e in events):raise APIError('api_generation_incomplete')
        value=json.loads((folder/'structured_reply.json').read_text());output=[]
        for call in value['calls']:
            args=json.loads(call['arguments_json'])
            if call['name'] not in schema['properties']['calls']['items']['properties']['name']['enum'] or not isinstance(args,dict):raise APIError('invalid_tool_call')
            output.append(dict(type='function_call',call_id='q0_'+uuid.uuid4().hex,name=call['name'],arguments=json.dumps(args,ensure_ascii=False)))
        result=dict(status='completed',model=self.model,output=output)
        atomic(folder/'response.json',result);return result
