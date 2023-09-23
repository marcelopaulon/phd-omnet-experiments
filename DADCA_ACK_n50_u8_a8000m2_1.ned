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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=1198.42,7927.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1597.81,7966.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2091.59,7987.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4170.00,7939.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4758.99,7809.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=500.45,7662.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1466.04,7551.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2597.28,7667.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3162.47,7718.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3780.03,7683.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5133.29,7751.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5491.79,7630.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=605.63,7324.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1002.79,7462.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2200.35,7343.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2614.89,7293.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3566.14,7332.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4246.18,7335.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=76.86,7142.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3000.30,7123.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4536.96,7026.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4910.66,7240.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5426.44,7139.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5948.47,7240.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=435.94,6973.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1302.60,6962.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1812.52,6844.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2454.26,6937.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4162.89,6888.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4964.64,6817.83");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5418.46,6775.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=103.57,6560.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=698.44,6678.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2147.63,6533.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3337.97,6741.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4600.20,6606.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1329.67,6455.23");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1659.08,6280.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2712.83,6398.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3475.25,6410.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3956.29,6501.64");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4293.00,6337.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4996.07,6428.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5382.69,6346.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5953.89,6345.73");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=458.92,6066.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1055.07,6059.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2239.87,6058.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3165.99,6070.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=102.42,5893.76");
    }

}

