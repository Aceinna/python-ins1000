
import json

def load_configuration(json_file):
    '''
    load properties from json_file
    returns: json properties when load successfully.
             None when load failed.
    '''
    try:
        with open(json_file) as json_data:
            return json.load(json_data)
    # except (ValueError, KeyError, TypeError) as error:
    except Exception as e:
        print(e)
        return None
