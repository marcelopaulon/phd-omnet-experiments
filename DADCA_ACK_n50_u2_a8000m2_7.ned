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

network DADCA_ACK_n50_u2_a8000m2_7
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
        @display("i=misc/sensor;p=653.78,7915.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1843.64,7843.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2836.36,7897.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3858.37,7975.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4444.76,7820.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5609.60,7895.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=149.23,7691.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1341.67,7679.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2254.20,7642.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3813.19,7618.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5193.72,7647.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=519.59,7483.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1217.87,7334.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1664.33,7473.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2726.77,7419.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3304.09,7493.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4283.70,7365.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=378.91,7102.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=803.67,7253.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1539.93,7127.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2940.94,7021.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3329.07,7079.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3740.59,7102.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4654.63,7217.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5094.22,7216.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=785.33,6779.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1947.89,6851.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2347.21,6866.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4041.28,6792.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5040.13,6812.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5630.92,6782.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1385.36,6656.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3031.17,6654.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3494.19,6552.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4597.44,6528.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1881.20,6454.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2721.42,6342.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5117.59,6373.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5523.87,6352.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=312.36,6041.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=695.27,6253.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1260.06,6235.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2324.84,6204.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3048.38,6195.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3443.22,6097.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3958.77,6068.18");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4609.47,6138.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5274.28,6035.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=789.39,5892.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1893.30,5896.43");
    }

}

