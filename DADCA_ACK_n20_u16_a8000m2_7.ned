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

network DADCA_ACK_n20_u16_a8000m2_7
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
        @display("i=misc/sensor;p=437.24,7817.18");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1070.68,7916.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1392.81,7776.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1930.53,7964.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2432.91,7816.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2884.28,7945.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3328.17,7757.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3715.20,7960.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2133.68,7600.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4341.43,7678.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4690.73,7651.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5551.33,7620.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1808.23,7391.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2883.02,7441.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3261.24,7373.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3867.94,7463.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4979.19,7448.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=154.87,7201.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1055.04,7242.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1466.32,7115.45");
    }

}

