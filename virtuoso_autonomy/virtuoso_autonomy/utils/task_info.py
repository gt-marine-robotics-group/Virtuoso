from ros_gz_interfaces.msg import ParamVec

def get_state(vec:ParamVec):

    for param in vec.params:
        if param.name == 'state':
            return param.value.string_value

def get_name(vec:ParamVec):

    for param in vec.params:
        if param.name == 'name':
            return param.value.string_value