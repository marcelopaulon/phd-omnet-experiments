package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u4_a8000m2_4
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=2834.69,7851.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3460.60,7805.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4402.09,7930.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=246.01,7537.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1112.16,7660.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1505.63,7646.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1971.33,7556.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2634.71,7552.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3813.10,7684.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4617.16,7636.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5224.18,7620.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2254.78,7342.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3139.30,7312.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3533.89,7443.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4291.75,7282.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5828.84,7425.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=388.94,7124.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=738.44,7143.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1203.56,7060.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3706.12,7078.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4844.21,7224.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5313.58,7141.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1538.72,6877.29");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2013.79,6995.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2572.06,7005.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3227.95,6906.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=440.60,6645.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1240.14,6552.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2064.29,6516.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3054.83,6515.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4086.41,6711.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4578.05,6710.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5035.46,6620.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5476.51,6516.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5828.17,6569.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1685.00,6425.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2690.56,6483.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3744.06,6443.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=158.21,6169.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=793.67,6176.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1398.58,6155.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3332.19,6262.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4407.90,6230.41");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5326.57,6090.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5903.23,6034.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1953.77,6004.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2498.23,5841.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2956.72,5775.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3601.34,5813.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5040.19,5884.10");
    }

}

