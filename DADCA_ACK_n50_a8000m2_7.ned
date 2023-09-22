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

network DADCA_ACK_n50_a8000m2_7
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
        @display("i=misc/sensor;p=545.23,7905.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1077.07,7938.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1582.34,7844.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4258.45,7879.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=171.29,7736.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1056.32,7501.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2128.16,7573.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2838.74,7662.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3292.60,7662.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5049.46,7602.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=204.89,7335.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1633.40,7392.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3651.03,7268.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4729.31,7376.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5473.27,7378.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5973.39,7283.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=853.97,7034.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1934.83,7121.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2605.91,7132.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4368.28,6992.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5057.22,7168.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=486.86,6820.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1238.73,6871.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2217.05,6849.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3385.83,6808.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3769.34,6852.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4737.12,6897.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5758.31,6782.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=93.91,6617.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=924.92,6609.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2838.28,6509.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4085.92,6596.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5214.42,6594.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=378.98,6328.76");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=819.23,6236.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1405.81,6333.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1911.65,6263.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2519.74,6263.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3357.40,6357.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4397.18,6235.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4777.40,6415.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2792.71,6008.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3754.52,6067.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5486.41,6112.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=641.90,5899.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1036.50,5926.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1642.39,5724.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2271.02,5746.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3283.27,5748.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4577.20,5698.62");
    }

}

