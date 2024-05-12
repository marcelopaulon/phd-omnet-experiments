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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=869.56,7960.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1309.66,7888.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2192.45,7926.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2816.23,7832.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3604.64,7928.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=247.05,7749.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1465.11,7564.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2031.31,7547.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2392.17,7539.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3137.89,7684.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3942.37,7695.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=796.65,7429.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2781.64,7272.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3345.64,7292.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3827.60,7265.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4467.37,7286.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=102.63,7088.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1365.68,7195.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1783.47,7195.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=553.44,6994.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1003.01,6843.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1480.61,6790.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2503.81,6875.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3045.22,6870.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4092.95,6987.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4594.90,6832.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4969.77,6987.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=598.86,6582.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2195.38,6649.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3366.96,6649.43");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3848.79,6609.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5191.42,6710.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5653.98,6582.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=197.72,6438.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=990.49,6485.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1473.77,6345.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2613.93,6469.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2993.94,6388.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4262.98,6363.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4784.50,6456.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=261.10,6090.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=853.12,6157.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1790.66,6136.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2433.82,6111.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3648.45,6105.18");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5066.45,6225.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3032.58,6015.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3901.24,5801.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4520.87,5848.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5720.85,5906.75");
    }

}

