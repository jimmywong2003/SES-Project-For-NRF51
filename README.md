# SES_Project_for_nRF51

##Segger Embedded Studio Project for nRF51 development



According to the bluetooth sig official web page (Source: Bluetooth SIG Email)

The Bluetooth Special Interest Group (SIG) recently announced their decision to deprecate and withdraw older versions of Bluetooth Core specifications from v2.0 to v4.1. Firstly, there is no need to panic. Secondly, here is what you need to know.

1) Customers who have already certified/listed their product with the Bluetooth SIG are not affected – once certified, always certified.
–  Additionally, none of this has any impact on Laird’s ability to produce and supply any of our Bluetooth modules.

2) Bluetooth v2.0 has been deprecated for some time now and is finally coming to an end. Products using a Bluetooth v2.0 radio (e.g. Laird’s TRBLU23-00200 or BT730) must be Bluetooth SIG listed before January 28, 2019. However, it is important to note that as the standard is already deprecated, it will mean a $25,000 listing fee as opposed to the usual $4,000/$8,000.

2) The biggest area for review is the mass deprecation of Bluetooth v2.1 – v4.1 on January 28, 2019 and complete withdrawal by July 1, 2020. Again, any certification/listing at these specification levels must be completed before the start of 2019 to keep costs at the usual $4,000/$8,000 listing fees.

Thus, if you need to use the nRF51 for the new bluetooth qualification, you SHOULD use the Softdevice S130v2.0.1 for the qualification approval.

## Nordic SDK version working with Bluetooth 4.2 (Softdevice S130 v2.0.1).

You can use either SDK 11.0.0 or SDK 12.3.0 (the last SDK working with NRF51) with S130 v2.0.1.


## Requirements
nRF5 SDK version 12.3.0
nRF51-DK 

To compile it, clone the repository in the \nRF5_SDK_12.3.0_d7731ad\ folder. If you download the zip, place each of the project folders of this repository into the \nRF5_SDK_12.3.0_d7731ad\ folder.

