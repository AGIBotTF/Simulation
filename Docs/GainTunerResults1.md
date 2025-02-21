# GAIN TUNER RESULTS FROM ARM.STL TEST 1


## Sinusoidal Joint Plots

This utility is used to help tune the gains of an Articulation.  Select the Articulation you would like to tune from the dropdown menu.

Gain tuning can be considered successful if near-perfect position tracking is observed in the sinusoidal gains test at the maximum velocities intended for your use case, and if reasonable behavior is observed in the step function gains test. Try running both tests to understand more.

|                                                                                                                                                                                                                             |                                                                                                                                                                                                                             |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXeUT0sLHyvc4MTTh3AwWhLtwluoA1z_L-JAk1IRIkLy3045XCJSsd7JcO_o79NwyQj01gv5ZeSdqREt8NQdZSy4BXPCYuFPKy1cftalNfFG0g3PHjN63ZxJIgWR_OS_mFZwFGnJYg?key=p0q-lc9JwWPKyk6l9eZiBNIp) | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdbSeBxPIne1as6C1yLuzyJvi07nypxcZzLR3Cue4hWk8wxZKPT3sowaOdbfO9iv2gGIzldQK4Ai3y6dTAh2-kM3Z5G9TmtQQnK31yC30Q11EYHDEPXfrma5ylLna5-CX8MGGua?key=p0q-lc9JwWPKyk6l9eZiBNIp)   |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXdhrVdz6J5pZwzVCiHFAVQStd43jc4hxO4KiGgiq4NLXh09fN-CAwJJNGIt0gy0dVSqord98E7ppCuQxpxAaRR4WWAZsu26uJSqbJe0LXmLJyot5Un9u5KJoyM1MzdW2QXeJbnF?key=p0q-lc9JwWPKyk6l9eZiBNIp)   | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXcTw8VNhGNLphc5wJLBX3Ba3CdITXtnCCOwES5c8yoFNASKkQ5wxxy_j11rdHGp5HJT2CKVlrH7MWH9pkY9tKORdSZYXb0xe3HG8-BZPxgIgiTpbGV1OXT5npVtoswiOC1YcA1rWw?key=p0q-lc9JwWPKyk6l9eZiBNIp) |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXeVhDYI-E0IbemYRRL97txHFOh89AKVYEsFbvxDeGIuq_sD8I-TR0QMPdGnyX1k-NMJuBY8waJLjiCUxVzzkPPFS25mOQVBhyq8GuZmLMlpSkMWSy4wyMM_TTlZKA5nhZfDqxriIA?key=p0q-lc9JwWPKyk6l9eZiBNIp) | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXfVsmaQzRNYSZdmiytAsgAIVdnwzu0R0hdEcwNKTEQdthj5EgSvcCdfrGKEfU6q_IhINLLxoaxoQa8ljV0_DgSp3SsV7rxR0bshNY54mXg-vnRHxAOfPky1ByrmmdB1cvbCIZCS?key=p0q-lc9JwWPKyk6l9eZiBNIp)   |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXd5wK91nhyzDBePQQy6Ugw5brbt968qUgGPSlpa8xcuaf8p2HZ7W9fvXFeinzw2qjD6H3Dka9KgJsLnJImDACY0UPROqKXRuYu0Xlvef0D36zG7SqLOQIBOuwCi8GBZR3esn4RkAw?key=p0q-lc9JwWPKyk6l9eZiBNIp) | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXfo-ah1_lYOQ8OsZFPw-TBj_-cbOqEnsIj9WaFI9cRf2LdzasLFPbouGQwXLAaCST5qq2OstJDAIMS5EgOGkXjA4fd-y1jvJFfdtU0uYr-gKW5Ms028gcuKKB01Ybk7tDZPNJxMrg?key=p0q-lc9JwWPKyk6l9eZiBNIp) |

Toni:

All lines are green which means that it perfectly matches the expected path.

**Future note** - if it goes wrong (red mismatch) the problem is either in the torques or the links

## Step Function Joint Plots

Unlike the sinusoidal gains test, perfect position tracking is not expected in this test. This test serves to validate whether an asset behaves reasonably when sent discontinuous joint commands. The duration of time that each joint is commanded to be at its high position is determined by approximating the time expected for that joint to get there if saturating either maximum velocity or effort. The results can be interpretted as reasonable if the position plots are smooth and the maximum positions are reached at roughly the same time as predicted.

The user should note that it is always better to send an Articulation a continuous path to fully control desired behavior. The purpose of this test is to give an intuitive sense for how the asset handles discontinuities that may arise through world collisions or programmatic mistakes. 

|                                                                                                                                                                                                                           |                                                                                                                                                                                                                             |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXd_tGlAtBjNiyTROIfPHRoJ-c1mLEj6fsAzbZuWbP8QS878f9qbnKPjzUHEtbSy_FFQsaZ78KEZwHNAb1GPncwI9vZQzjlQzYwNBNgVTvdivStqdeU24t-kEFd4jkZ8RgbNTeku?key=p0q-lc9JwWPKyk6l9eZiBNIp) | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXe27ESycCzGdHrlIHTZlRz4O67Ib4_TSdZVc76UXoPgqGyWBgRhZNwbsT_KTahHAqp5RmCm0fClH8lpLW0RiDMjJ1stn_Z6KJK4Dh_c7tNCDjUxsh4WvNzPEjTd14ZeoOcdXe5TNw?key=p0q-lc9JwWPKyk6l9eZiBNIp) |
| ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXeFxQMgm4J_QmrQhU_fy7jJs5ERxeifioT54mNbTHJ3t-Cx1QVm2eXmP1hRwwcWDtRz0OkLxDLjJVLJ4jjREQ0Yd1V_-4rXnqz-mlWGwenFJZ8CTdddZJFL1PrDaijVh3HPGIRN?key=p0q-lc9JwWPKyk6l9eZiBNIp) | ![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXfBiT6GHodAedx_z_lj8cPRS1rk2SBi2m63fhAyFHRQWL9ZCIIYWjrIB6fBKUdZMSERYCkFuqrnPWnJ3pwQe-N-5X5ktvfOBdXHsM4s363PmbqInW-X2S2TUNCFagcJqbzwbGsK0g?key=p0q-lc9JwWPKyk6l9eZiBNIp) |

\


Toni:

This mismatch probably comes from the fact that the joints don’t have enough ‘power’ (powerful enough) compared to the example mass I gave to the parts. Like I said, that causes the lower joints (the ones that hold more parts above them) to slow down. 

Either way this is a test for “discontinuous movement” so with **careful code handling SMOOT movement** of the arm, it won't be a problem - should be tested in reality!

(TLDR - this page is not important its just for future reference if the tuned model will have errors)

