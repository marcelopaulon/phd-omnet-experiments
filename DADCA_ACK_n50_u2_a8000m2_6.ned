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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=709.56,7832.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1064.25,7932.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1631.56,7862.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2297.67,7804.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3163.44,7966.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3773.43,7976.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4871.40,7864.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5504.26,7923.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5839.80,7807.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2885.53,7596.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4045.71,7668.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4711.57,7523.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=476.02,7480.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1088.49,7481.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1489.42,7273.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1840.59,7382.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2465.99,7439.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3518.72,7413.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4302.52,7277.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5306.72,7354.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5721.61,7436.67");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2120.29,7098.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3162.90,7011.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3651.94,7015.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=819.91,6953.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1540.15,6844.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2623.05,6906.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4032.43,6972.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4550.25,6985.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5101.09,6994.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5514.25,6995.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=489.51,6751.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=969.10,6626.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2198.47,6617.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4292.70,6603.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4787.83,6664.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5232.68,6645.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=21.78,6380.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=752.87,6281.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1352.28,6452.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1868.39,6469.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2447.08,6343.69");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2878.06,6394.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3293.11,6425.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3773.62,6339.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5573.76,6503.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1612.08,6032.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4022.15,6043.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4731.14,6228.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5100.41,6244.75");
    }

}

