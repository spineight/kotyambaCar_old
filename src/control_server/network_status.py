import subprocess 

def get_command_center_ip():
  ''' in case if command_center not avaiable throws Exception '''
  try:
    command_center_ip = subprocess.check_output(["getent", "ahosts", "commandCenter.local"]).split()[0]
    return command_center_ip
  except Exception as e:
    # https://realpython.com/python-exceptions/
    raise Exception("commandCenter.local not available")
  