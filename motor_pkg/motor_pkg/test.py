import importlib.util

module_name = 'Motorcontrol'
module_spec = importlib.util.find_spec(module_name)
if module_spec is None:
    print(f"Module {module_name} not found")
else:
    print(f"Module {module_name} found at {module_spec.origin}")
