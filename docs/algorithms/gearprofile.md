
involute gears
==============

$$
\newcommand{\mat}[1]{\begin{pmatrix}#1\end{pmatrix}}
\newcommand{\norm}[1]{\left\|#1\right\|}
$$


![everything comes from the rack](/schemes/gear-from-rack.png)

## involute curve

Those are very important for involute gears, check at [wikipedia for more details](https://en.wikipedia.org/wiki/Involute)

- involute for any $ gamma $ curve (with parameter $t$):
	$$
	\theta(t) = \gamma(t) + \gamma'(t) (t_0 - t)
	$$

- parametric equation of a circle (with radius r): 	
	$$
	\gamma(t) = r \mat{ cos(t) \\ sin(t) }
	$$
	
- 

$ \implies $ involute of circle (with radius r): 	
$$
\theta(t) =  r \mat{ cos(t) \\ sin(t) } + r (t_0-t) \mat { -sin(t) \\ cos(t) }
$$

![involute of circle](/schemes/gear-circle-involute.png)

#### few remarks

An involute is symetric over $t_0$: 
- $ \theta([t_0, +\infty[) $	is an involute developed by a direct rotation
- $ \theta(]-\infty, t_0]) $	is an involute developed by a reverse rotation



## a gear tooth

### parameters

- $ s = $ gear step (distance on the primitive)
- $ z = $ number of tooths on the gear
- $ \alpha = $ pressure angle (angle of the rack tooths)
- $ h = $ rack half height = $ \text{total height} \over 2 $
- $ e = $ (offset) amount to shift up the top and bottom of the rack, it does not change the tooth sides.
- $ x = $ fraction of the rack above the primitive (close to 0.5), usually $ = 0.5 + 2 {e \over p}  tan(alpha) $

#### useful circles

- $ p = $ primitive radius $ = {s z \over 2 \pi}  $
- $ c = $ base circle radius $ = p \cos(\alpha) $
- $ p - h + e = $ bottom circle radius
- $ p + h + e = $ top circle radius

### curves

The contact that transmits the torque is always oriented toward the rack tooth normal (angle $alpha$).
The only solution for this is to have the contact point moving on an axis from the contact between both primitives, and oriented along $alpha$

![contact points](/schemes/gear-contact-curve.png)

The contact curve is then an involute of circle.

$$
 \theta_0(t) = \text{contact curve} = c \mat{\cos(t) \\ \sin(t)} + c (t_0-t) \mat{-\sin(t) \\ \cos(t) }
$$

As the tooth is moving in/out of the gear, its opposite corner to the contact point may collide with a next tooth. This interference force us to remove material from the tooth, using the interference curve.

![interference point](/schemes/gear-interference-curve.png)

The interference curve is then an involute of circle with a radial offset.

$$
\theta_i(t) = \text{interference curve} = p \mat{cos(t) \\ sin(t)} + p (t_i-t) \mat{-sin(t) \\ cos(t)} - (h+e) \mat{cos(t) \\ sin(t)}
$$
![curves of tooths identified](/schemes/gear-curves-identified.png)



### start points

![curves start points](/schemes/gear-curves-start.png)

If we consider a period starting from the middle of the tooth top, there is two places our curves start (one each tooth). We can define them relatively to the crossing point between the primitive circle and the involute.

Crossing point between primitive circle and involute:	$ t_s = \begin{cases} {m \over p} {x \over 2}  \\  {m \over p} {{2-x} \over 2}  \end{cases} $


#### $ t_0 $

Can be determined relatively to $ t_s $ which is the place (but not the parameter) where $ theta_0 $ reachs radius $ p $ :

$$
\norm{ \theta_0(t) } = p  \iff   p^2 = \norm{\theta_0(t)}^2 = c^2( 1 + (t-t_0)^2 )    \iff   t - t_0 = \sqrt{({p \over c})^2 - 1}    =   tan(\alpha)
$$


we just need to take the angle for the obtained $ t $
$$
t_0 - t_s = angle( \theta(t_0 + tan(\alpha)) - \theta(t_0) )
$$

#### $ t_i $

$ theta_i $ starts at the very moment the tooth offset is radial:

![gear offset radial](/schemes/gear-offset-radial.png)

$$
t_i - t_s = atan({h-e \over p}  tan(\alpha))
$$

#### intersection between $ theta_0 $ and $ theta_i $

To know when does $ theta_i $ stop and when $ theta_0 $ start, we need to know the intersection point between them. As both curves are defined by different points of the rack at different times during the rotation, the two curves must have different parameters.

$$
\begin{align}
\text{intersection between } \theta_0 \text{ and } \theta_i 
&\iff \theta_0(t_1) = \theta_i(t_2)  \\
&\iff \theta_0(t_1) - \theta_i(t_2) = 0
\end{align}
$$


The equation becomes quite complicated, so we can use the [Newton's method](https://en.wikipedia.org/wiki/Newton%27s_method) to solve it

$$
\begin{align}
f(t_1, t_2) = \theta_0(t_1) - \theta_i(t_2) 
&=	c \mat{cos(t_1) \\ sin(t_1)} + c (t_0-t_1) \mat{-sin(t_1) \\ cos(t_1)} \\
	&- p \mat{cos(t_2) \\ sin(t_2)} + p (t_i-t_2) \mat{-sin(t_2) \\ cos(t_2)} \\
	&- (h-e) \mat{cos(t_2) \\ sin(t_2)}
\end{align}
$$

- Its [jacobian matrix](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant):

  $$
  \begin{align}
  J(f) 
    &= \mat{
         {\partial f \over \partial t_1} 
       & {\partial f \over \partial t_2}}  \\
    &= \mat{ 
  	-c*(t_0 - t_1) \mat{cos(t_1) \\ sin(t_1)}
  	& p*(t_i - t_2) \mat{cos(t_2) \\ sin(t_2)} + (h-e) \mat{-sin(t2) \\ cos(t_2)}
  	}
  \end{align}
  $$
  

- The newton method consist of just iteratively doing	 (until the necessary precision is reached)

	$$
	\mat{t_1 \\ t_2}   \leftarrow  \mat{t_1 \\ t_2} - J^-1 (f)(t_1, t_2) * f(t_1, t_2)
	$$
	


To make sure it will converge to the desired solution, we need to take an initial state in the convex zone of the solution:

- $t_1$ on the primitive circle: 	$ t_{1,text(initial)} = t_0 - tan(\alpha)$
- $t_2$ on the base circle: 	$ t_{2,text(initial)} = t_i + {\sqrt{ c^2 - (p-h+e)^2 } \over p}  $

It ends with the parameters $ t_1, t_2 $ of $ theta_0, theta_i $ at the intersection.

- $ \theta_i $ must stop at the intersection at $ t_2 $
- $ \theta_0 $ must start at the intersection at $ t_1 $

#### stop point $t_3$ of $ \theta_0 $

- in case $ theta_0 $ hits the top circle

	$ t_3 - t_0 = {\sqrt{ (p+h+e)^2 - c^2 } \over c} $

- in case $ theta_0 $ hits the involute of the next period

	We know that it will cross on the period-start axis $ (O,x) $, so
	
	$$
	\theta_0(t_3) . y = 0  \iff 0 = sin(t_3) + cos(t_3) * (t_0-t_3) = f(t) 
	$$
	A simple [Newton's method](https://en.wikipedia.org/wiki/Newton%27s_method) solves this with initial state on the top circle
