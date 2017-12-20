class A:
    def __init__(self):
        print("I'm A")

class B(A):
    def __init__(self):
        A.__init__(self)
        print("But also I'm B")


b = B()
