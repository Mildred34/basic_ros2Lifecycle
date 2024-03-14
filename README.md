# Basic ROS2 Lifecycle package

## Les bases
* *Qu'est-ce qu'une lifecycle node ?* \
Un noeud sur lequel, on peut à travers un noeud que j'appelerais *manager* \
gérer la vie des noeuds lifecycles via 4 états:
* *Unconfigured* \
Le noeud vient d'être créé, il n'est pas encore utilisable.
C'est publishers, services, serveurs d'actions... ne sont pas encore configurés.
* *Inactive* \
Les publishers, services, serveurs d'actions... du noeud sont maintenant configurés
mais non utilisable. \
En théorie non utilisable, mais sur les lifecyclepython, on a que les publishers qui sont
optimisé en lifecycle via *LifecyclePublisher*. \
En c++, ça ne doit pas être le cas
* *Active* \
Notre noeud vit maintenant sa meilleure vie, il peut travailler tranquille
* *Finalized* \
Son travail est terminé. On détruit tout publishers, services, serveurs d'actions...

## Tutoriel Python
Dans l'exemple Python de ce tutoriel, nous retrouvons 4 paquets:
* [manager](./src/manager) - Noeud qui va des services va gérer le cycle de vie des noeuds suivants
* [lifecycle_listener](./src/lifecycle_listener/) Noeud qui va communiquer avec *lifecycle_talker* \
Il sera composé d'un subscriber sur le topic du noeud suivant.
* [lifecycle_talker](./src/lifecycle_talker/) Un talker rien de plus basique qui va écrire au listener
* [lifecycle_talker2](./src/lifecycle_talker2/) Un second talker qui sera lancé quand les deux derniers noeuds auront terminé leur job.

Le [launch file](./src/manager/launch/allinone.launch.py) va tout d'abord lancé le *manager* avec 
les noeuds *lifecycle_talker* et *lifecycle_listener*. \
Quand leur job sera terminé, ils vont le dire aux manager qui va les shutdown.

Dans le launch file, on précise que lorsqu'ils ont atteint l'état shutdown, qu'on les détruit.
A la destruction, du noeud principal du talker/listener process, \
on vient lancer un second launch file composé du noeud *lifecycle_talker2*.

Qui de même lorsqu'il a fini de travaillé, on vient le shutdown puis le détruire.\
A la fin du travail de ce dernier, on shutdown le process.

## Tutoriel C++
Voir la bibliographie sur le sujet.

# Bibliographie:
## Composition
* [Publication - Impact de la composition ROS sur les systèmes Robotiques](https://arxiv.org/pdf/2305.09933.pdf) \
La composition étant le fait d'avoir plusieurs noeud sur un seul process. \
C'est un autre sujet que les noeuds lifecycle à part entière mais intéressant.
* [Dynamics composition en C++](https://betterprogramming.pub/introduction-to-ros-2-and-dynamic-composition-a-gateway-to-advanced-robotics-782b459215ee)

## Python
* [Demo python officiel lifecycle Ros 2 rolling](https://github.com/ros2/demos/tree/rolling/lifecycle_py)
* [How to Use ROS 2 Lifecycle Nodes - Tutorial](https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes)
* [Exemple de .launch avec ontransitionevent/shutdown](https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py#L74)
* [Calling a ROS2 service from the callback of another service](https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5) 
* [Exemple de launch file avec les lifecycle nodes](https://github.com/ros2/launch_ros/blob/6370c127868a5056a8a02c9412c59bebdaefcf81/launch_ros/examples/lifecycle_pub_sub_launch.py#L59)
* [Exemple d'un projet complet lifecycle avec IHM](https://robotics.stackexchange.com/questions/102991/ros-2-how-to-start-and-stop-a-node-from-a-python-script)
* [Explications des lifecycles nodes](https://design.ros2.org/articles/node_lifecycle.html)

* [Exemple d'un projet complet lifecycle avec IHM](https://answers.ros.org/question/412104/ros-2-how-to-start-and-stop-a-node-from-a-python-script/) \
Le même que l'autre exemple j'ai l'impression. Mais peut-être qu'il y a des meilleurs commentaires.

## C++
* [Exemple de noeuds Lifecycle en C++ sur Rolling](https://github.com/ros2/demos/blob/rolling/lifecycle/src/lifecycle_listener.cpp)
* [Lifecycle demo project](https://github.com/thehummingbird/robotics_demos/tree/main/lifecycle_node)
* [Components Management with ROS2 Lifecycle Nodes](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-8-simplify-robotic-software-components-management-with-ros2-5fafa2738700)