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
        @display("i=misc/sensor;p=352.69,7880.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=951.07,7811.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2781.13,7986.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3819.77,7977.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4665.07,7765.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1445.33,7702.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2076.33,7618.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2419.22,7691.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3373.63,7597.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4303.35,7711.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5717.17,7530.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=781.20,7426.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1135.10,7272.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2774.97,7377.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3183.44,7274.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3574.62,7294.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4033.51,7474.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4468.02,7395.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5165.80,7413.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=134.79,7222.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1725.70,7229.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2439.81,7096.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4547.56,7022.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5535.42,7161.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5973.61,7150.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=226.63,6764.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=804.00,6979.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1211.13,6788.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1864.04,6868.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3216.02,6872.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3743.60,6937.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4059.62,6783.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5320.05,6828.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=873.60,6581.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1649.13,6586.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2309.99,6703.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2716.44,6601.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3477.60,6570.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4990.07,6611.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5889.32,6530.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=506.03,6298.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1991.19,6442.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4389.42,6464.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5382.71,6391.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1115.77,6189.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1485.83,6041.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2338.99,6078.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3085.42,6096.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3801.54,6251.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4830.18,6060.15");
    }

}

