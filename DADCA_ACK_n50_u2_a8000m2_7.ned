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
        @display("i=misc/sensor;p=153.93,7924.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=842.27,7987.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1187.31,7762.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2834.18,7962.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4075.84,7822.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4476.20,7948.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=483.00,7656.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1510.16,7540.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2237.82,7655.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2669.51,7521.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3390.89,7588.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4500.28,7553.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5155.01,7691.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=134.51,7475.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=785.86,7267.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1939.61,7309.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3091.90,7395.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4009.10,7457.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4771.64,7277.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5916.53,7406.14");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=543.15,7014.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1281.28,7133.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2523.98,7184.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3456.25,7230.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5358.15,7206.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=168.93,6903.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1016.29,6869.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1695.35,6914.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2049.82,6782.94");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2951.14,6918.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3427.84,6860.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3756.90,6991.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4312.94,6918.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4846.68,6916.23");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1531.50,6599.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2511.13,6688.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3958.97,6657.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4640.66,6597.97");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5273.24,6689.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5860.41,6723.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=254.14,6398.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=882.11,6481.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2216.58,6460.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2673.15,6318.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3136.71,6454.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3652.38,6372.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4894.86,6300.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5481.25,6366.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=513.40,6151.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1239.03,6192.83");
    }

}

