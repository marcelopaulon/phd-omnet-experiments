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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=311.02,7808.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1782.89,7896.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2224.51,7778.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3110.38,7922.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3769.13,7868.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4929.18,7932.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5845.43,7875.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=958.15,7680.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2630.76,7517.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3084.12,7543.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4430.11,7573.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5330.18,7711.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=37.59,7289.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=398.88,7288.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1303.28,7473.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2087.26,7415.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3659.31,7445.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3993.50,7282.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5684.88,7305.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1053.79,7164.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1713.89,7243.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2078.87,7015.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2427.20,7152.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3229.42,7206.33");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4372.44,7057.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5003.70,7121.63");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=116.97,6926.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=704.00,6799.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1280.57,6795.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2965.43,6939.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3614.50,6834.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5704.32,6938.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=367.04,6546.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1816.09,6701.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2441.69,6622.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3288.78,6585.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4199.88,6617.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4949.32,6651.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=885.35,6383.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1352.17,6281.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2712.68,6315.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3652.74,6425.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4077.35,6288.36");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4784.91,6302.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5404.08,6454.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=112.45,6244.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=540.60,6149.86");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1995.90,6260.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3043.76,6186.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4484.90,6086.52");
    }

}

