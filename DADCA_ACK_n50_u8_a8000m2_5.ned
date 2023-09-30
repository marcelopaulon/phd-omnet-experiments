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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=758.85,7913.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1303.55,7952.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1675.00,7987.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2014.18,7799.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2854.83,7914.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3540.13,7903.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4055.39,7845.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5365.18,7828.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5867.54,7989.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=453.20,7564.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2539.25,7548.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4483.59,7732.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5011.73,7556.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=903.14,7375.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1767.30,7358.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2139.26,7344.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3187.28,7428.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3734.51,7471.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4404.60,7374.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5597.14,7422.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2844.99,7249.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5626.43,7072.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5985.83,7181.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=428.62,6901.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1150.53,7007.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2048.27,6930.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3223.92,6836.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3703.79,6835.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4345.93,6964.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5020.88,6784.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=919.89,6690.89");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1440.03,6572.07");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1748.92,6747.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2569.15,6707.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=510.75,6507.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1813.19,6317.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2239.51,6467.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3157.04,6425.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4156.42,6499.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4665.21,6333.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5454.41,6515.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=270.20,6237.19");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=886.62,6089.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1313.65,6142.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2711.75,6204.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3662.13,6191.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5312.07,6165.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5781.61,6198.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=329.12,5845.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1865.72,5959.08");
    }

}

