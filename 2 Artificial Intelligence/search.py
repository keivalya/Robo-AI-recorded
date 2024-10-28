import sys

class Node():
  def __init__(self, state, parent, action):
    self.state = state
    self.parent = parent
    self.action = action

class StackContainer():
  def __init__(self):
    self.container = []
  
  def add(self, node):
    self.container.append(node)
  
  def contains_state(self, state):
    for node in self.container:
        if node.state == state:
            return True
    return False
  
  def empty(self):
    return len(self.container) == 0

  def remove(self):
    if self.empty():
      raise Exception("Container is empty.")
    else:
      node = self.container[-1]
      self.container = self.container[:-1]
      return node

class QueueContainer(StackContainer):
  def remove(self):
    if self.empty():
      raise Exception("Container is empty.")
    else:
      node = self.container[0]
      self.container = self.container[1:]
      return node