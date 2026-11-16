import json,time,hashlib,subprocess,pathlib,yaml
r=pathlib.Path('/home/orangepi/FC01_navigation')
d=json.loads((r/'status.json').read_text());s=d.get('state') or {}
props=subprocess.check_output(['systemctl','show','fc01-navigation.service','-p','MainPID','-p','ActiveState','-p','DropInPaths','-p','WorkingDirectory','-p','ExecStart'],text=True)
controllers=[]
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  if args and pathlib.Path(args[0]).name=='fc01_system_node':
   controllers.append(dict(pid=int(p.name),exe=str((p/'exe').resolve()),sha256=hashlib.sha256((p/'exe').read_bytes()).hexdigest(),argv=args))
 except (OSError,ValueError):pass
result=dict(time=time.time(),service=props,controllers=controllers,controller_count=len(controllers),status=d.get('status'),feedback_current=d.get('feedback_current'),state={k:s.get(k) for k in ('phase','reason','armed','landed','at_rest','thrust_body','rc_target_height','rc_navigation_mode_slot','visual_navigation_enabled','arm_enabled','flight_actions_enabled','takeoff_enabled','fc01_entry_version','fc01_config_path','fc01_config_source_path')},runtime=yaml.safe_load((r/'runtime.yaml').read_text()),runtime_sha256=hashlib.sha256((r/'runtime.yaml').read_bytes()).hexdigest())
print(json.dumps(result,indent=2))
