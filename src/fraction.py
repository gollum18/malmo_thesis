#############################################
# Name: fraction.py
# Since: 9/12/2017
# Mdified: 9/12/2017
# Purpose: Allows for the representation of 
#   fractions.
# Author: Christen Ford
#############################################

from __future__ import division
import math
import random

def commonize(f1, f2):
    """
    Returns a tuple containing two normalized fractions, that is they both have
    the same denominator while maintaining their original ratios.
    :param f1: Fraction one.
    :param f2: Fraction two.
    :return: A tuple containing two new commonized fractions in the same order as f1, f2.
    """
    m = lcm(f1.denominator, f2.denominator)
    m1, m2 = m/f1.denominator, m/f2.denominator
    return (Fraction(f1.numerator*m1, f1.denominator*m1), 
            Fraction(f2.numerator*m2, f2.denominator*m2))

def lcm(m, n):
    m, n = abs(m), abs(n)
    return (m/gcd(m, n))*n

def gcd(m, n):
    if n == 0:
        return m
    return gcd(n, m % n)

class Fraction:
    
    def __init__(self, numerator, denominator):
        """
        Creates a fraction from an integer numerator and integer denominator.
        :param numerator: The numerator of the fraction.
        :param denominator: The denominator of the fraction.
        :return: N/A
        """
        if denominator == 0:
            raise ValueError("Error! Denominator cannot be zero!")
        self.numerator = numerator
        self.denominator = denominator

    def __str__(self):
        """
        Creates a human readable string representation of the fraction.
        :return: String representing the fraction.
        """
        return "{0}/{1}".format(self.numerator, self.denominator)

    def __hash__(self):
        return hash((self.numerator, self.denominator))

    def __eq__(self, other):
        fractions = commonize(self, other)
        return (fractions[0].numerator == fractions[1].numerator and 
            fractions[0].denominator == fractions[1].denominator)

    def __ne__(self, other):
        return not self.__eq__(self, other)

    def __lt__(self, other):
        fractions = commonize(self, other)
        return fractions[0].numerator < fractions[1].numerator

    def __gt__(self, other):
        fractions = commonize(self, other)
        return fractions[0].numerator > fractions[1].numerator

    def __add__(self, other):
        fractions = commonize(self, other)
        self.numerator = fractions[0].numerator + fractions[1].numerator
        self.denominator = fractions[0].denominator
        self.simplify()

    def __mul__(self, other):
        self.numerator *= other.numerator
        self.denominator *= other.denominator
        self.simplify()

    def __sub__(self, other):
        fractions = commonize(self, other)
        self.numerator = fractions[0].numerator - fractions[1].numerator
        self.denominator = fractions[0].denominator
        self.simplify()

    def __div__(self, other):
        self.numerator *= other.denominator
        self.denominator *= other.numerator
        self.simplify()

    def get_numerator(self):
        return self.numerator

    def get_denominator(self):
        return self.denominator

    def set_numerator(self, numerator):
        self.numerator = numerator

    def set_denominator(self, denominator):
        if denominator == 0:
            raise ValueError("Error! Denominator cannot be zero!")

    def as_mixed(self):
        return ("{0} + {1}/{2}".format(int(math.floor(self.numerator/self.denominator)), 
                                       self.numerator % self.denominator, self.denominator))
    
    def simplify(self):
        """
        Simplifies the fraction.
        :return: N/A
        """
        div = gcd(self.numerator, self.denominator)
        self.numerator /= div
        self.denominator /= div

    @staticmethod
    def add(f1, f2):
        """
        Adds two fractions.
        :param f1: The first fraction.
        :param f2: The second fraction.
        :return: A simplified fraction that is the result of adding f1 and f2.
        """
        fractions = commonize(f1, f2)
        f = Fraction(fractions[0].numerator + fractions[1].numerator, fractions[0].denominator)
        f.simplify()
        return f

    @staticmethod
    def divide(f1, f2):
        """
        Divides two fractions.
        :param f1: The first fraction.
        :param f2: The second fraction.
        :return: A simplified fraction that is the result of dividing f1 and f2.
        """
        f = Fraction(f1.numerator * f2.denominator, f1.denominator * f2.numerator)
        f.simplify()
        return f

    @staticmethod
    def equals(f1, f2):
        """
        Determines if two fractions are equal.
        :param f1: The first fraction.
        :param f2: The second fraction.
        :return: True if the two fractions are equal, False otherwise.
        """
        fractions = commonize(f1, f2)
        return (fractions[0].numerator == fractions[1].numerator and 
            fractions[0].denominator == fractions[1].denominator)

    @staticmethod
    def subtract(f1, f2):
        """
        Subtracts two fractions.
        :param f1: The first fraction.
        :param f2: The second fraction.
        :return: A simplified fraction that is the result of subtracting f1 and f2.
        """
        fractions = commonize(f1, f2)
        f = Fraction(fractions[0].numerator - fractions[1].numerator, fractions[0].denominator)
        f.simplify()
        return f

    @staticmethod
    def multiply(f1, f2):
        """
        Multiplies two fractions.
        :param f1: The first fraction.
        :param f2: The second fraction.
        :return: A simplified fraction that is the result of multiplying f1 and f2.
        """
        f = Fraction(f1.numerator * f2.numerator, f1.denominator * f2.denominator)
        f.simplify()
        return f
    
if __name__ == '__main__':
    f = Fraction(2, 3)
    print f.as_mixed()