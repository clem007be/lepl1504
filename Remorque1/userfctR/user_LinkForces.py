# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0

    linkG_id = mbs_data.link_id['Amortisseur_RoueG']
    linkD_id = mbs_data.link_id['Amortisseur_RoueD']
    
    if (identity == linkG_id or identity == linkD_id):
        K = mbs_data.user_model['Amortisseur_Arriere']['K']
        D = mbs_data.user_model['Amortisseur_Arriere']['D']
        Z0 = mbs_data.user_model['Amortisseur_Arriere']['Z0']
        Flink = K*(Z-Z0) + D*Zd

    return Flink
