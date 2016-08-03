function p = log_likelihood(mu, sigma)

p = 0.918938533204673 - log(sigma) - 0.5*(mu/sigma)^2;

end