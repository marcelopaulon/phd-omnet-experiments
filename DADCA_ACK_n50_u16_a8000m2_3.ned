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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=632.45,7898.35");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1151.93,7819.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1941.02,7927.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3078.11,7889.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3490.03,7925.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4596.50,7989.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4944.40,7895.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5668.54,7885.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=840.64,7505.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2253.68,7746.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2631.59,7575.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4045.72,7714.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5260.28,7585.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5734.20,7514.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=461.80,7407.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1343.90,7285.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3093.30,7270.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3589.91,7500.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4263.50,7425.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4626.98,7406.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2109.48,7134.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3897.81,7062.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4379.97,7063.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5093.20,7235.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=298.49,6908.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=883.96,6884.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1828.86,6882.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2478.62,6932.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3061.58,6819.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3437.32,6860.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5426.00,6881.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5968.97,6991.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1539.94,6573.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3764.93,6563.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4213.08,6557.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5043.52,6748.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5917.22,6643.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=51.06,6320.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=700.97,6491.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1942.59,6431.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3086.36,6420.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4622.28,6466.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5263.61,6456.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5697.17,6322.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=312.47,6074.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1258.47,6211.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2510.81,6239.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3564.58,6099.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4031.76,6216.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4821.98,6169.11");
    }

}

