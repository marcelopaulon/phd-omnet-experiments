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

network DADCA_ACK_n50_u8_a8000m2_2
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
        @display("i=misc/sensor;p=979.41,7815.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1457.46,7816.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2751.45,7761.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3461.49,7788.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3882.76,7824.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4689.33,7871.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5317.28,7968.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5678.88,7773.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=675.34,7605.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2168.30,7677.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5201.70,7612.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=265.37,7301.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1151.32,7366.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1714.88,7481.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2038.55,7348.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2728.12,7309.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3097.39,7273.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3499.62,7283.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4293.22,7457.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4840.82,7477.72");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=640.72,7049.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1750.79,7131.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3857.81,7147.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4817.60,7023.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5194.67,7117.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5812.61,7192.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=235.76,6867.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1391.89,6962.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2231.08,6840.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2668.25,6820.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3080.83,6790.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3433.60,6848.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4248.77,6847.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=827.78,6599.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1571.83,6536.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3988.06,6546.25");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4878.67,6545.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5287.42,6717.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=180.12,6448.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=572.63,6315.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1243.27,6383.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1921.01,6511.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2858.25,6492.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3324.43,6510.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3657.97,6404.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2486.18,6143.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3932.86,6157.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4512.24,6263.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5481.84,6264.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=318.52,5891.31");
    }

}

