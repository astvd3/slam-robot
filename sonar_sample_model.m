function vec_out=sonar_sample_model(sonar_model, sample_size)

sample_phit=ones(1,floor(sonar_model{1}.p_hit*sample_size));
sample_short=2*ones(1,floor(sonar_model{1}.p_short*sample_size));
sample_rand=3*ones(1,floor(sonar_model{1}.p_rand*sample_size));
sample_max=4*ones(1,floor(sonar_model{1}.p_max*sample_size));

vec_out=[sample_phit sample_short sample_rand sample_max];