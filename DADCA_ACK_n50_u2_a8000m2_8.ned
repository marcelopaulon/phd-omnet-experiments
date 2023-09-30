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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=896.36,7753.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2026.54,7862.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2374.58,7799.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3799.44,7928.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4552.07,7926.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5072.75,7990.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5345.43,7765.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5755.36,7957.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1360.94,7507.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2977.92,7596.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4025.62,7542.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5007.66,7565.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5953.04,7575.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1884.41,7338.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2509.86,7429.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3423.80,7453.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4678.46,7434.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5299.87,7289.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5663.34,7358.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=490.24,7177.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1531.47,7112.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2249.80,7142.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5934.20,7134.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1083.39,6948.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2074.25,6834.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2749.67,6975.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3403.37,6983.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3900.40,6995.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4336.13,6931.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4779.17,6926.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5158.50,6907.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=204.19,6583.22");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=622.53,6525.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1626.74,6650.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2331.15,6551.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4503.30,6523.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4896.91,6594.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5677.70,6542.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1098.88,6474.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1441.01,6308.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1924.60,6340.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2866.56,6494.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3310.77,6338.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4077.77,6461.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5211.07,6436.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2321.29,6181.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2666.17,6055.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3679.41,6194.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4327.09,6138.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5419.46,6036.87");
    }

}

