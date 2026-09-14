// Executes the shipped inline JavaScript with a fake DOM and fake fetch only.
// No web service, camera, MAVLink endpoint, or real browser is contacted.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname,'../src/uav_preview/web/index.html'),'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script); // syntax-check the whole shipped script

class Element {
  constructor(){this.checked=false;this.style={};this.listeners={};this.classList={toggle(){}};}
  addEventListener(type,fn){(this.listeners[type]??=[]).push(fn);}
  closest(){return null;}
  blur(){}
}
const nodes=new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>[m[1],new Element()]));
const winEvents={};
const calls=[];
let focused=true;
let state={active:true,handoff:{run_id:'run-identity-000000000001',revision:0,ready:true,authorized:false,phase:'HOLDING'}};
function stateCopy(){return JSON.parse(JSON.stringify(state));}
const document={
  hidden:false,activeElement:null,hasFocus:()=>focused,
  getElementById:id=>nodes.get(id),querySelectorAll:()=>[],querySelector:()=>null,
  addEventListener(){},createElement:()=>new Element(),
};
const context=vm.createContext({document,HTMLElement:Element,crypto:{randomUUID:()=> 'client-identity-0000000001'},
  performance:{now:()=>Date.now()},console,URL,requestAnimationFrame(){},setInterval(){},setTimeout,
  window:{addEventListener:(event,fn)=>(winEvents[event]??=[]).push(fn),confirm:()=>{throw Error('blocking modal not allowed in handoff');}},
  fetch:async (url,options)=>{
    if(!options?.method)return new Promise(()=>{}); // startup state/video reads stay inert
    const body=JSON.parse(options.body||'{}');calls.push({url,body});
    let token;
    if(url.endsWith('/authorize')) {
      state.handoff.authorized=true;state.handoff.ready=false;state.handoff.revision++;
      token='permission-token-000000000001';
    } else if(url.endsWith('/revoke')||!body.foreground||!body.confirmed) {
      state.handoff.authorized=false;state.handoff.revision++;
    }
    return {ok:true,json:async()=>({ok:true,state:stateCopy(),token,message:'fake response'})};
  },
});
vm.runInContext(script,context);
const run=s=>vm.runInContext(s,context);
const click=async id=>{for(const f of nodes.get(id).listeners.click||[])await f();};
const adopt=()=>{context.testState=stateCopy();run('adoptHandoff(testState)');};

(async()=>{
  nodes.get('real-action-confirm').checked=true;
  adopt();assert.equal(nodes.get('smooth-handoff-authorize').disabled,false);
  await run('sendHandoffInput()');
  assert.equal(calls.at(-1).body.token,null);
  assert.equal(calls.at(-1).body.keys_released,true);
  await click('smooth-handoff-authorize');
  await Promise.resolve();await Promise.resolve();
  assert.equal(run('handoffToken'),'permission-token-000000000001');
  assert.equal(run('keyboardControlEnabled'),false);
  assert.ok(calls.some(c=>c.url.endsWith('/authorize')));
  assert.ok(calls.every(c=>c.url.startsWith('/api/local-takeoff/keyboard/')));

  // A delayed preauthorization GET may not erase a new permission.
  run("adoptHandoff({active:true,handoff:{run_id:handoffRun,revision:0,authorized:false}})");
  assert.notEqual(run('handoffToken'),null);
  await run('sendHandoffInput()');
  assert.equal(calls.at(-1).body.pitch,0);
  run("simPressed.add('KeyW');simChannels.pitch=2000");
  await run('sendHandoffInput()');assert.equal(calls.at(-1).body.pitch,1);
  assert.equal(calls.at(-1).body.keys_released,false);
  run('simPressed.clear()'); // channel animation has not run yet
  await run('sendHandoffInput()');assert.equal(calls.at(-1).body.pitch,0);
  assert.equal(calls.at(-1).body.keys_released,true);
  await click('smooth-handoff-revoke');
  assert.equal(run('handoffToken'),null);
  assert.ok(!calls.some(c=>/flight\/|keyboard-control\/enable/.test(c.url)));

  state.handoff.ready=true;adopt();await click('smooth-handoff-authorize');
  await Promise.resolve();await Promise.resolve();
  focused=false;document.hidden=true;
  for(const callback of winEvents.blur)await callback();
  await run('sendHandoffInput()');
  assert.equal(calls.at(-1).body.foreground,false);
  assert.equal(calls.at(-1).body.pitch,0);
  assert.equal(run('handoffToken'),null);
  focused=true;document.hidden=false;adopt();
  assert.equal(run('handoffToken'),null); // focus return never reauthorizes

  // This tab cannot send for an already-authorized different tab.
  state.handoff.authorized=true;state.handoff.revision++;adopt();
  const previous=calls.length;await run('sendHandoffInput()');assert.equal(calls.length,previous);
  state.handoff.run_id='run-identity-000000000002';state.handoff.authorized=false;adopt();
  assert.equal(run('handoffSequence'),0);assert.equal(run('handoffToken'),null);
  // Takeoff-only disables input and permission even after an old response arrives.
  const takeoffOnlyCount=calls.length;
  run('adoptHandoff({active:true,keyboard_handoff_enabled:false,handoff:null})');
  assert.equal(nodes.get('smooth-handoff-authorize').disabled,true);
  assert.equal(nodes.get('smooth-handoff-revoke').disabled,true);
  assert.match(nodes.get('smooth-handoff-state').textContent,/键盘交接已关闭/);
  assert.doesNotMatch(nodes.get('smooth-handoff-state').textContent,/3\.0 s/);
  adopt(); // stale permissive response cannot re-enable this page
  await run('sendHandoffInput()');await click('smooth-handoff-authorize');
  assert.equal(calls.length,takeoffOnlyCount);
  assert.equal(run('handoffRun'),null);assert.equal(run('handoffToken'),null);
  // Strict keyboard diagnostics must not override the separate fixed-hover gate.
  context.baselineState={navigation_safety:{block_reason:'strict missing flow',block_reasons:['strict missing flow'],
    local_takeoff_block_reason:'',local_takeoff_block_reasons:[],local_takeoff_warnings:['flow unverified']}};
  assert.equal(run('localTakeoffNavigation(baselineState).blockReason'),'');
  assert.equal(run('localTakeoffNavigation(baselineState).blockers.length'),0);
  assert.equal(run('localTakeoffNavigation(baselineState).warnings[0]'),'flow unverified');
  context.baselineState.navigation_safety.local_takeoff_block_reason='position invalid';
  context.baselineState.navigation_safety.local_takeoff_block_reasons=['position invalid'];
  assert.equal(run('localTakeoffNavigation(baselineState).blockReason'),'position invalid');
  assert.equal(run("localTakeoffNavigation({navigation_safety:{block_reason:'old backend denial'}}).blockReason"),'old backend denial');
  console.log('Handoff frontend: session, input, revoke, focus, stale-state, isolation and takeoff-only checks passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
