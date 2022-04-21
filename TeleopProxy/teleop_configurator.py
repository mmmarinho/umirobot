# Server information
POST_RET_HEADER = 'port_request:'
SERVER_LOCATION = "teleop-gateway.qlin.me"
REQUEST_PORT = 39999
REQUEST_URL = "http://{}:{:d}/request".format(SERVER_LOCATION, REQUEST_PORT)


def read_server_key(path):
    with open(path, 'r') as f:
        data = f.read()
    return data


def _check_header(ret_str):
    if ret_str.find(POST_RET_HEADER) != 0:
        return False
    return True


def get_auth_string(ret_str):
    if not _check_header(ret_str):
        print("Got wrong return header:", ret_str)
        return None
    body = ret_str[len(POST_RET_HEADER):].lstrip()
    if body.split(",")[0] != "OK":
        print("Reply not OK:", ret_str)
        return None
    return body.split(",")[1]


def get_return_port(ret_str):
    if not _check_header(ret_str):
        print("Got incorrect return header:", ret_str)
        return None
    body = ret_str[len(POST_RET_HEADER):].lstrip()
    if body.split(",")[0] != "port":
        print("Reply not OK:", ret_str)
        return None
    return int(body.split(",")[1])
