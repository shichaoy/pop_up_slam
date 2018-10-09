/**
 * @file References.h
 * @brief iSAM references in doxygen format.
 * @author Michael Kaess
 * @version $Id: References.h 6353 2012-03-26 22:16:29Z kaess $
 *
 * Copyright (C) 2009-2013 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen,
 * Nicholas Carlevaris-Bianco and John. J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/** @page Bibliography Bibliography

A list of iSAM-related publications in BibTeX format.

@section bib_journal Journal Papers

@verbatim

% most relevant publication for covariance recovery
@Article{Kaess09ras,
  author =       {M. Kaess and F. Dellaert},
  fullauthor =   {Michael Kaess and Frank Dellaert},
  title =        {Covariance Recovery from a Square Root Information
                  Matrix for Data Association},
  doi =          {10.1016/j.robot.2009.06.008},
  journal =      {Journal of Robotics and Autonomous Systems (RAS)},
  volume =       57,
  issue =        12,
  month =        dec,
  pages =        {1198-1210},
  year =         2009,
}

% most relevant publication for iSAM
@Article{Kaess08tro,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank Dellaert},
  title =        {{iSAM}: Incremental Smoothing and Mapping},
  journal =      {IEEE Trans. on Robotics (TRO)},
  volume =       {24},
  number =       {6},
  pages =        {1365-1378},
  month =        dec,
  year =         2008,
}

% most relevant publication for Square Root SAM
@Article{Dellaert06ijrr,
  author =       {F. Dellaert and M. Kaess},
  fullauthor =   {Frank Dellaert and Michael Kaess},
  title =        {Square {Root} {SAM}: Simultaneous Localization and Mapping
                  via Square Root Information Smoothing},
  journal =      {Intl. J. of Robotics Research (IJRR)},
  volume =       25,
  number =       12,
  pages =        {1181-1204},
  month =        dec,
  year =         2006,
}

@endverbatim

@section bib_other Other Publications

@verbatim

% iSAM2 and Bayes tree journal paper
@Article{Kaess12ijrr,
  author =       {M. Kaess and H. Johannsson and R. Roberts and V. Ila
                  and J.J. Leonard and F. Dellaert},
  fullauthor =   {Michael Kaess and Hordur Johannsson and Richard
                  Roberts and Viorela Ila and John J. Leonard and Frank
                  Dellaert},
  title =        {{iSAM2}: Incremental Smoothing and Mapping Using the
                  {B}ayes Tree},
  journal =      {Intl. J. of Robotics Research (IJRR)},
  volume =       31,
  issue =        2,
  pages =        {217-236},
  month =        feb,
  year =         2012,
}

% Incremental Dog-Leg
@InProceedings{Rosen12icra,
  author =       {D.M. Rosen and M. Kaess and J.J. Leonard},
  fullauthor =   {David M. Rosen and Michael Kaess and John
                  J. Leonard},
  title =        {An Incremental Trust-Region Method for Robust Online
                  Sparse Least-Squares Estimation},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {St. Paul, MN},
  month =        may,
  year =         2012,
}

% visual SLAM
@InProceedings{McDonald11ecmr,
  author =       {J. McDonald and M. Kaess and C. Cadena and J. Neira
                  and J.J. Leonard},
  fullauthor =   {John McDonald and Michael Kaess and Cesar Cadena and
                  Jos\'{e} Neira and John J. Leonard},
  title =        {6-{DOF} Multi-session Visual {SLAM} using Anchor
                  Nodes},
  booktitle =    {European Conference on Mobile Robots (ECMR)},
  address =      {Orebro, Sweden},
  pages =        {69-76},
  month =        sep,
  year =         2011,
}

% iSAM2
@InProceedings{Kaess11icra,
  author =       {M. Kaess and H. Johannsson and R. Roberts and V. Ila
                  and J.J. Leonard and F. Dellaert},
  fullauthor =   {Michael Kaess and Hordur Johannsson and Richard
                  Roberts and Viorela Ila and John J. Leonard and
                  Frank Dellaert},
  title =        {{iSAM2}: Incremental Smoothing and Mapping with
                  Fluid Relinearization and Incremental Variable
                  Reordering},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {Shanghai, China},
  pages =        {3281-3288},
  month =        may,
  year =         2011,
}

% Bayes Tree
@InProceedings{Kaess10wafr,
  author =       {M. Kaess and V. Ila and R. Roberts and F. Dellaert},
  fullauthor =   {Michael Kaess and Viorela Ila and Richard Roberts
                  and Frank Dellaert},
  title =        {The {B}ayes Tree: An Algorithmic Foundation for
                  Probabilistic Robot Mapping},
  booktitle =    {Intl. Workshop on the Algorithmic Foundations of
                  Robotics (WAFR)},
  address =      {Singapore},
  month =        dec,
  year =         2010,
}

% anchor nodes
@InProceedings{Kim10icra,
  author =       {B. Kim and M. Kaess and L. Fletcher and J. Leonard
                  and A. Bachrach and N. Roy and S. Teller},
  fullauthor =   {Been Kim and Michael Kaess and Luke Fletcher and
                  John Leonard and Abe Bachrach and Nicholas Roy and
                  Seth Teller},
  title =        {Multiple Relative Pose Graphs for Robust Cooperative
                  Mapping},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {Anchorage, Alaska},
  pages =        {3185-3192},
  month =        may,
  year =         2010,
}

% iSAM + covariances, for journal paper see Kaess08tro
@InProceedings{Kaess07icra,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank
                  Dellaert},
  title =        {{iSAM}: Fast Incremental Smoothing and Mapping with
                  Efficient Data Association},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {Rome, Italy},
  pages =        {1670-1677},
  month =        apr,
  year =         2007,
}

% first iSAM paper, for journal paper see Kaess08tro
@InProceedings{Kaess07ijcai,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank
                  Dellaert},
  title =        {Fast Incremental Square Root Information Smoothing},
  booktitle =    {Intl. Joint Conf. on Artificial Intelligence (IJCAI)},
  address =      {Hyderabad, India},
  pages =        {2129-2134},
  month =        jan,
  year =         2007,
}

% first Square Root SAM paper, for journal paper see Dellaert06ijrr
@InProceedings{Dellaert05rss,
  author =      {F. Dellaert},
  title =       {Square {Root} {SAM}: Simultaneous Location and
                 Mapping via Square Root Information Smoothing},
  booktitle =   {Robotics: Science and Systems (RSS)},
  year =        2005,
}

@endverbatim

*/
