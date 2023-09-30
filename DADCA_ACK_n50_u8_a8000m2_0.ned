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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=211.65,7826.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1054.00,7981.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1825.60,7810.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3662.57,7905.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4630.01,7820.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5611.53,7929.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=722.46,7668.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1542.99,7546.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2493.38,7539.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3077.04,7672.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3987.73,7693.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=179.94,7395.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2136.00,7420.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3518.69,7467.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3868.76,7282.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5152.17,7484.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5733.34,7406.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1404.61,7221.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1805.13,7111.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2515.02,7079.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2992.09,7197.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4252.14,7183.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4841.15,7201.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5446.70,7194.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=336.29,6778.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=799.52,6935.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3483.77,6804.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3854.90,6906.66");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4526.16,6958.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5435.81,6791.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1112.87,6758.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1516.62,6695.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1865.57,6735.30");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2540.58,6655.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3044.51,6758.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4190.46,6703.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4806.23,6553.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5849.22,6750.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=552.37,6448.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1451.73,6272.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2273.38,6294.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3718.99,6497.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4452.49,6433.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5916.60,6403.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=222.92,6030.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2014.99,6023.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2867.75,6187.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3233.38,6029.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3856.43,6110.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4731.61,6098.29");
    }

}

