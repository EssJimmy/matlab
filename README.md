# Cálculo de control dinámico y por velocidad de un robot
## Declaración de parámetros iniciales y de variables
Declaración de parametros iniciales, las variables  indican variables, las variables  indican distancias no variables y  el número de grados de libertad o de joints (considerando un robot no sobreactuado).

$R_k^0$ se inicializa como 1 para facilidad, z y  lo mismo para después modificarse adecuadamente.

$J$ corresponde al jacobiano final, $J_v$ corresponde al jacobiano lineal, y $J_\omega$ corresponde al jacobiano angular, $J_{v\sigma}$ corresponde a un parametro auxiliar para el cálculo de $J_{vc}$.

$\dot q$ e $I$ son representaciones auxiliares de  y de los tensores de inercia, cuyos valores los conocemos a la hora de calcular el modelo dinámico.

## Cálculo de matrices de Denavit-Hartenberg del robot
Para lo que sirve el primer for es para calcular Denavit-Hartenberg, los demás for sirven para limpiar la matriz, dado que MATLAB no es capaz de calcular $sin(\pi) = 0$ , por ejemplo, aquí también hacemos la obtención de $z$ y $\sigma$.

## Cálculo del Jacobiano linear y angular del robot
Dependiendo si el joint es revoluto o prismático, tenemos cambios en el jacobiano angular y el jacobiano linear, si el joint es prismático, el jacobiano linear toma $z_i$, y el angular es un vector de 0, si el joint es revoluta, el jacobiano linear toma el valor de $z_i \times (\sigma_n - \sigma_i)$ y el jacobiano angular toma el valor de $z_i$, también aquí se implementa la lógica para obtener $J_{vc}$ y $J_{wc}$.

## Matriz de inercias y Matriz de Coriolis
Utilizando $J_{vc}$ y $J_{wc}$ podemos calcular la matriz de inercias sin tener que meternos con multiplicaciones por tensores de inercia, que es la primera parte de este código, la segunda parte es calcular la matriz de coriolis utilizando la fórmula que implica los símbolos de Christoffel, esto se multiplica por $\dot q$ y obtenemos $C(q,\dot q)$ de una forma más sencilla, dado que $\dot q$ es simbólica en este código, no sabremos hasta obtener los parámetros según el tipo de controlador que usemos.

## Energía potencial y su gradiente
Ayudándonos de una variable auxiliar para calcular $J_{vc}$ somos capaz de calcular como se vería la energía potencial de cada joint, para el gradiente simplemente utilizamos la derivada de P en q o $\frac{\partial P}{\partial q_i}$.

## Energía cinética
Habiendo calculado la energía potencial, y la matriz de inercias, para calcular la energía cinética es muy sencillo, siguiendo la formula encontrada en las notas, tenemos que $\mathcal K = \frac{1}{2} {\dot q}^\top D \dot q$.

## $\tau$
$K_p$ y $K_d$ son matrices dadas por el problema, por lo que aquí las estoy señalando como syms para que no haya problema, pero deben ser cambiadas, lo mismo sucede con $q_d$, que es un vector dado que ya se conoce, $\tau$ se calcula de esa forma y la inversa de la matriz de inercias nos sirve para calcular la segunda derivada del vector q, $\ddot q$.

# Algo de teoría de control por velocidad
## Jacobiano
Formado por el jacobiano lineal y jacobiano angular, su -ésima columna describe la velocidad lineal y angular del -ésimo joint.
## Tensor de inercia 
El tensor de inercia describe cómo se distribuye la masa de un cuerpo en relación con su sistema de coordenadas, relaciona las fuerzas y momentos aplicados a un cuerpo con sus velocidades lineales y angulares.
## Matriz de inercias
La matriz de inercia $D(q)$ refleja cómo la masa está distribuida en las diferentes partes del  robot y cómo esa distribución afecta la respuesta dinámica del sistema a las fuerzas aplicadas. Es simétrica, positiva definida y es en general dependiente de la configuración.
Matriz de Coriolis  La matriz de Coriolis es una representación matemática utilizada en dinámica de sistemas físicos, especialmente en el contexto de la mecánica de cuerpos rígidos. Describe la relación entre las fuerzas y momentos aplicados a un cuerpo y las velocidades angulares y lineales del mismo. Hay dos formas comunes de representar la matriz de Coriolis:

1. Forma Expandida: 
En esta forma, la matriz de Coriolis se representa como una matriz cuadrada que relaciona las fuerzas generalizadas ($\tau$)  con las velocidades generalizadas ($\dot q$), donde $\tau$ es el vector de fuerzas generalizadas y $\dot q$ es el vector de velocidades generalizadas. La relación se expresa mediante la ecuación:

$$
  \tau = C(q, \dot q)
$$

2. Forma Compacta: 
La forma compacta de la matriz de Coriolis se puede expresar mediante la siguiente ecuación diferencial:

$$
  \tau = C(q, \dot q) + V(q, \dot q)
$$

En esta ecuación, $V(q, \dot q)$ representa las fuerzas centrifugas y gravitacionales, y $\tau$ es el vector de fuerzas generalizadas. La relación describe cómo las fuerzas generalizadas dependen de las velocidades generalizadas, la matriz de Coriolis y las fuerzas adicionales. Es simétrica

# Algo de teoría de control linear / no linear
Existen tres tipos de robots, totalmente actuados, subactuados y sobreactuados (corresponde al número de joints utilizado para actuar sobre n grados de libertad), y en general pueden ser rígidos o flexibles. Existen dos tipos de actuación, actuación directa e indirecta, esto significa que los motores pueden estar directamente en el joint, o sobre la base, respectivamente. Y tenemos dos tipos de transmisores del movimiento; mediante poleas y/o cadenas y mediante harmonic drives una tecnología mucho más nueva.

Para robots totalmente actuados, la tarea de control es trivial siempre y cuando tengamos $q$ y $\dot q$, dado que podemos aplicar control dinámico o por velocidad.

Otra parte de los robots son los observadores, estos son aquellos que miden $q$ y $\dot q$, y también somos capaces de medir su "calidad" con los siguientes parámetros

1. Razón de convergencia
2. Complejidad
3. Robustez: ruido de medida e incertidumbre en el modelo.

Lo anterior aplica si el sistema es lineal, el problema es, que no todos los sistemas son lineales, y no podemos abarcar un problema no lineal, utilizando la serie de Taylor en primer grado para linealizar el sistema, por lo que hemos hecho lo siguiente:

1. Definir el punto de operación
2. Linealizar el "entorno" del punto de operación

Las principales dificultades de esto es obviamente la no linealidad (la ganancia preprogramada o gain scheduling), la flexibilidad de algunos joints y la incertidumbre paramétrica. Además de obviamente considerar cosas como la gravedad y demás, estos serían los principales retos del control no linear.
## Dificultades matemáticas del control no linear
Lo principal es que es muy difícil la resolución de sistemas no lineares, esto dado a que no tenemos representaciones frecuenciales (como la transformada de Laplace o de Fourier), por lo que no tenemos una función de transferencia, así que no podemos poner a la entrada en términos de la entrada o viceversa, de una forma directa.

Otra dificultada es la estabilidad, nos concentraremos en dos términos de estabilidad principal, BIBO estable y asintóticamente estable, pero la que más nos interesa, es la estabilidad en el sentido de Lyapunov.
