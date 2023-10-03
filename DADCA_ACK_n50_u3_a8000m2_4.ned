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

network DADCA_ACK_n50_u3_a8000m2_4
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
        @display("i=misc/sensor;p=458.74,7949.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1001.60,7935.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1763.33,7946.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3245.88,7882.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3894.88,7855.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4375.31,7897.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4847.73,7934.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5412.75,7803.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5889.25,7784.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=668.44,7510.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1526.31,7582.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1972.49,7513.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2327.43,7683.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2818.36,7674.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3152.33,7517.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3515.35,7646.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5627.44,7520.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=146.60,7331.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1073.17,7479.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3452.22,7269.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4017.91,7503.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4667.41,7434.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5217.86,7466.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1319.08,7223.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1766.71,7035.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2281.03,7076.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2603.72,7252.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=938.04,6995.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1443.16,6830.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3048.45,7008.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3801.37,6802.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4189.71,6961.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4748.85,6828.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5190.12,6964.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5570.97,6889.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=152.01,6661.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=620.48,6650.23");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2069.20,6586.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2883.19,6664.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=997.53,6323.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1645.38,6385.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2533.99,6335.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3278.39,6472.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4195.92,6504.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4570.13,6457.32");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5120.24,6481.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5485.45,6407.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=167.77,6209.04");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1942.26,6121.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2923.94,6242.30");
    }

}

