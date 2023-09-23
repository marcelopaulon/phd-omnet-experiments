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

network DADCA_ACK_n50_u3_a8000m2_1
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
        @display("i=misc/sensor;p=114.05,7805.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=466.74,7775.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1076.71,7804.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2404.65,7860.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3212.46,7836.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3853.12,7814.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4307.19,7826.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4935.78,7862.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5945.01,7899.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1749.88,7730.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2056.39,7528.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5332.04,7645.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=424.06,7365.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2434.38,7496.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2980.42,7478.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3628.17,7393.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4534.18,7357.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4900.59,7450.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5253.60,7299.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5959.57,7501.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1093.06,7209.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1572.34,7070.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2060.79,7065.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2609.01,7174.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3244.41,7193.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3925.91,7102.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4960.55,7066.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5732.43,7202.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2879.23,6842.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3500.85,6776.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4177.67,6773.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5445.32,6835.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=63.01,6726.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=611.97,6735.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1078.61,6730.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1735.72,6603.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2287.98,6597.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4776.40,6754.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5223.37,6521.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5961.28,6604.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=406.37,6268.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=797.61,6356.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2852.59,6354.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3269.12,6297.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3665.56,6423.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5653.83,6397.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=76.98,6111.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1175.92,6147.40");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1554.40,6250.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1865.87,6078.14");
    }

}

