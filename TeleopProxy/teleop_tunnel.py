import subprocess
import os
import shutil
import time
import tempfile


class temporary_copy(object):
    def __init__(self, original_path):
        self.original_path = original_path

    def __enter__(self):
        self.tmp_dir = tempfile.mkdtemp()
        base_path = os.path.basename(self.original_path)
        self.path = os.path.join(self.tmp_dir, base_path)
        shutil.copy2(self.original_path, self.path)
        if not os.path.exists(self.path):
            raise RuntimeError("Error creating temporary key file")
        return self.path

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.remove(self.path)
        shutil.rmtree(self.tmp_dir)


class TeleopProxyTunnel:
    def __init__(self,
                 remote_username, key_path,
                 remote, remote_open_port,
                 local_forward_target, local_forward_port):
        """
        ssh tunnel handler,
        NOTE: need to test cross platform
        :param remote_username: ssh username
        :param key_path: ssh key file path
        :param remote: server url
        :param remote_open_port: server open port
        :param local_forward_target: local forward address
        :param local_forward_port: local forward to port
        """
        self._process = None
        self._cmd = ["ssh", "-N"]

        self.keypath_obj = temporary_copy(key_path)

        self._cmd_opt = ["-o", "StrictHostKeyChecking=no",
                         "-o", "ServerAliveInterval=60",
                         "-R", '*:{:d}:{:s}:{:d}'.format(remote_open_port,
                                                         local_forward_target,
                                                         local_forward_port),
                         remote, "-l", remote_username, "-p", "22"]

        # print(self._cmd_opt)
        self.remote_open_port = remote_open_port

    def open(self):
        """
        Open tunnel with  initiated parameter
        """
        with self.keypath_obj as key_path:
            self._process = subprocess.Popen(self._cmd + ["-i", key_path] + self._cmd_opt)
            time.sleep(1)

        return self.remote_open_port

    def is_alive(self):
        """
        Check if tunnel is alive
        :return: true if tunnel is alive
        """
        poll = self._process.poll()
        if poll is None:
            return True
        return False

    def close(self):
        """
        close tunnel
        """
        if self._process.poll() is None:
            self._process.terminate()

    def __del__(self):
        if self._process.poll() is None:
            self._process.terminate()
