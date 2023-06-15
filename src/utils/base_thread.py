import sys
import threading
import traceback


class BaseThread(threading.Thread):
    def __init__(self, exception_bucket=None, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=True):
        if name is None:
            name = target.__name__
        super().__init__(group=group, target=target, name=name, args=args, kwargs=kwargs, daemon=daemon)
        self.exception_bucket = exception_bucket
        self.exit_code = None
        self.exception = None
        self.exc_traceback = ''
        self.ret_val = None

    def run(self):
        try:
            self._run()
        except Exception as e:
            self.exception = e
            self.exit_code = 1
            self.exc_traceback = traceback.format_exc()
            self.exception_bucket.put(sys.exc_info())
            raise e
        else:
            self.exit_code = 0

    def _run(self):
        try:
            if self._target is not None:
                self.ret_val = self._target(*self._args, **self._kwargs)
        except Exception as e:
            raise e
        finally:
            del self._target, self._args, self._kwargs
