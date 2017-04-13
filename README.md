# autonomousCAR
Computer Vision, CNN, NVIDIA model...

    // create layer of 10 linear neurons (no activation function by default)
    {type:'fc', num_neurons:10}
    // create layer of 10 neurons that use sigmoid activation function
    {type:'fc', num_neurons:10, activation:'sigmoid'} // x->1/(1+e^(-x))
    {type:'fc', num_neurons:10, activation:'tanh'} // x->tanh(x)
    {type:'fc', num_neurons:10, activation:'relu'} // rectified linear units: x->max(0,x)
    // maxout units: (x,y)->max(x,y). num_neurons must be divisible by 2.
    // maxout "consumes" multiple filters for every output. Thus, this line
    // will actually produce only 5 outputs in this layer. (group_size is 2)
    // by default.
    {type:'fc', num_neurons:10, activation:'maxout'} 
    // specify group size in maxout. num_neurons must be divisible by group_size.
    // here, output will be 3 neurons only (3 = 12/4)
    {type:'fc', num_neurons:12, group_size: 4, activation:'maxout'}
    // dropout half the units (probability 0.5) in this layer during training, for regularization
    {type:'fc', num_neurons:10, activation:'relu', drop_prob: 0.5}
