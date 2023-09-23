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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=433.64,7844.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2709.57,7865.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=127.46,7611.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1244.88,7579.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3232.76,7749.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3939.38,7655.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5594.71,7670.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5975.11,7587.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=601.99,7409.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2070.53,7370.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2539.56,7443.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4671.06,7403.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1222.66,7217.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2746.12,7116.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3553.57,7253.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5468.60,7230.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5813.77,7163.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=333.53,6915.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=875.37,6770.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1588.25,6966.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2379.95,6771.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3282.51,6919.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4171.27,6955.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4863.27,6925.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1483.10,6593.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1953.06,6627.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3875.32,6524.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5231.00,6559.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5823.88,6714.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=412.96,6447.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2582.89,6466.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3499.02,6284.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4368.65,6439.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5497.76,6294.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1078.39,6177.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2004.82,6212.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2967.80,6230.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4929.02,6152.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=225.01,5846.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=749.94,5834.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1391.29,5937.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2510.99,5805.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3534.86,5933.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3882.34,5857.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5713.41,5862.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1127.84,5570.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1481.01,5534.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1806.50,5734.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3041.31,5541.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4660.77,5741.78");
    }

}

