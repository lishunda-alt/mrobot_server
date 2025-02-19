# -------------- 优先级队列 --------------
import heapq


class PriorityQueue(object):
    def __init__(self) -> None:
        self._queue = []
        self._arg_queue = []

        self._index = 0
        self._number = 0

    def push(self, item, priority, *_args):
        heapq.heappush(self._queue, (-priority, self._index, item))
        heapq.heappush(self._arg_queue, (-priority, self._index, _args))
        self._index += 1
        self._number += 1

    def pop(self):
        if self._number < 1:
            return
        self._number -= 1
        _args = heapq.heappop(self._arg_queue)[-1]
        return heapq.heappop(self._queue)[-1](*_args)

    def empty(self):
        return self._number == 0
