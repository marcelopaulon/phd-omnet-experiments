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

network DADCA_ACK_n50_u3_a8000m2_2
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
        @display("i=misc/sensor;p=28.49,7855.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=449.19,7752.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1505.42,7812.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1880.60,7904.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4332.76,7780.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5417.69,7930.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=911.16,7567.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2094.86,7546.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2625.11,7690.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3305.82,7713.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3993.81,7673.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5666.37,7525.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=121.32,7432.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1386.88,7406.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2812.05,7287.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3300.26,7264.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3699.05,7262.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5006.51,7489.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=346.31,7154.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=753.23,7036.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1192.53,7056.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1824.21,7105.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2468.94,7131.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4613.49,7219.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5219.11,7053.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5997.95,7082.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=72.44,6915.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1498.81,6789.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3053.04,6937.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4138.93,6828.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4799.90,6826.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5591.35,6962.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=315.26,6614.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1035.99,6659.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1928.92,6680.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2839.53,6583.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3647.03,6690.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4523.69,6563.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5826.12,6685.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=225.75,6268.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=692.80,6412.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2514.85,6383.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4164.46,6441.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5225.16,6454.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1122.00,6049.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1491.43,6224.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1873.85,6113.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3196.24,6255.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4917.78,6088.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5433.08,6037.26");
    }

}

