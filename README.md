# Searching-In-Grid-World
Interactive implementation of Graph Algorithms like Depth First search, Uniform Cost Search, and A* Search in Python

To run the project, download the project and type in the command line:

python search.py

A GUI similar to the below image will be generated. You can select one from the above mentioned algorithms and see in real-time how it explores the search space. New random state spaces and be created by pressing the Random button. The algorithm always starts from the red dot and tries to search for the blue dot or the goal node. It stops searching once it reaches the goal node.  

![alt text](http://github.com/Kalpit-Vadnerkar/Searching-In-Grid-World/blob/main/pasted%20image%200.png?raw=true)

As all action costs in the grid world are unit costs, uniform cost search behaves like breadth-first search with both methods returning optimal solutions (breaking ties in the priority queue may result in visually different minimal cost routes). You can verify this by implementing BFS by simply changing the data structure used in your DFS implementation to expand the open list nodes in a FIFO manner. Of course, when the step costs vary, BFS cannot guarantee optimal paths. You can change the cost function in search_app.py to verify this.

One should see that in general A* finds the optimal solution (slightly) faster than UCS. Note once more that the way you break ties in the priority queues may lead to different minimal cost paths for the two methods. By setting the heuristic distance to 0, the A* search effectively behaves like uniform cost search. By comparing the two search algorithms, you should be able to see the advantages of heuristics search on certain scenarios.

You can find implementations of Queue, Stack (LIFO Queue), and PriorityQueue data structures in search_app.py. It is important to use these data structures for your open lists as they are needed for compatibility with the rest of the code. A Set data structure is also provided for the closed list. The code automatically visualizes the states in the open and closed list. States on the open list are colored white, and expanded nodes are colored red.
