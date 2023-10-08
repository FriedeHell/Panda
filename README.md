# Panda
 
### Setup mit des FCI über folgendes Video: 
[https://www.youtube.com/watch?v=91wFDNHVXI4&t=371s](https://www.youtube.com/watch?v=91wFDNHVXI4&t=371s)

```jsx
sudo ping 192.168.1.11 -i 0,001 -D -c 10000 -s 1200
```

## Matlab Installation

Matlab auf der Mathworks Homepage herunterladen: [Mathworks Homepage](https://de.mathworks.com/) 

In beliebigen Ordner enpacken

In Ordner der enptackten Datei wechseln. Bsp: 

```matlab
cd Downloads
cd .....
sudo ./install
```

## Franka Matlab Installation

Franka Matlab nach Anleitung installieren: [Franka Matlab Dokumentation](https://frankaemika.github.io/docs/franka_matlab/index.html) 

Disable Frequency Scaling: [Disable Frequency Sacling Documentation](https://frankaemika.github.io/docs/troubleshooting.html#disabling-cpu-frequency-scaling)

Ansonsten einfach der Franka Matlab Dokumentation folgen

## GitHub Installation

GitHub Desktop nach folgender Anleitung installieren: [GitHub Desktop Ubuntu Anleitung](https://gist.github.com/berkorbay/6feda478a00b0432d13f1fc0a50467f1#file-github_desktop_ubuntu-sh-L5) 

# Matlab Franka Emika Panda installation für die Simulation

```matlab
roboticsAddons
```

Robotics System Toolbox Library Data herunterladen

Joint Limits: 

[Robot and interface specifications — Franka Control Interface (FCI)  documentation](https://frankaemika.github.io/docs/control_parameters.html)

# Matlab Zeichensammlung

Matlab Projekt schließen, ohne matlab zu beenden: `close`

```matlab
>> proj = openProject("C:\Users\fried\MATLAB\Projects\Panda\Panda.prj");
```
