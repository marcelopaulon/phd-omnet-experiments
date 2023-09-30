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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=1098.09,7820.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1615.83,7958.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2563.88,7780.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3755.87,7997.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4277.81,7970.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4940.11,7757.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=777.82,7613.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1598.45,7535.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3232.64,7569.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4000.18,7696.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5472.22,7667.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=410.54,7457.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1003.52,7262.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2156.26,7495.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2701.00,7315.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3776.38,7427.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4402.92,7430.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4744.01,7334.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=151.79,7033.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3524.91,7046.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4154.83,7134.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5202.88,7081.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5581.56,7172.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5978.53,7185.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1371.85,6908.28");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1869.51,6881.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2351.08,6921.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2882.61,6937.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3977.79,6829.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4510.53,6827.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4860.77,6989.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5798.18,6814.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=115.13,6545.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1064.40,6711.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3477.45,6698.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5082.34,6714.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=560.35,6465.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1677.48,6463.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2142.39,6432.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2520.23,6466.56");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3068.34,6424.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4373.09,6397.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4935.37,6301.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=826.73,6057.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1237.17,6112.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2700.67,6150.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3594.27,6261.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4080.12,6163.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=196.73,5934.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1734.75,5964.05");
    }

}

