# Hemisphere V500 Ros2

Package ROS2 permettant la lecture des données fournies par le récepteur GNSS Hemisphere V500

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (FISE 2023)
Basé sur le travaux réalisé par Pierre NARVOR sous ROS

## Git Structure :

* :file_folder: [/include](include) : **dossier contenant les headers**
* :file_folder: [/msg](msg) : **dossier contenant les messages personnalisés**
* :file_folder: [/python](python) : **Utilisation du protocole mavlink**
* :file_folder: [/src](src) : **dossier contenant les fichiers source**
* :file_folder: [/test](test) : **dossier contenant les différents tests (ici munu_io)**

## Technologies :

* Ubuntu 20.04
* C++
* ROS2 foxy

## Dépendances :

Cette bibliothèque dépend de la bibliothèque munu_io développée par Pierre NARVOR autour de boost::asio, disponible [ici](https://github.com/godardma/munu_io)


## Lancement

* Cloner le repo munu_io
* Ouvrir un terminal dans le dossier munu_io et taper les commandes suivantes :
```bash
mkdir build && cd build && cmake .. && make
sudo make install
```
* Cloner le repo dans un workspace ROS2 Foxy
* Build le package et le sourcer:
```bash
colcon build --packages-select hemisphere_v500
. install/setup.bash
```
* Le noeud peut-être lancé via la commande :
```bash
ros2 run hemisphere_v500 hemisphere_v500_node
```
Par la suite le noeud pourra être inclus dans un launcher afin de limiter le nombre de terminaux à ouvrir


