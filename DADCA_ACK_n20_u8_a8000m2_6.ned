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

network DADCA_ACK_n20_u8_a8000m2_6
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
        @display("i=misc/sensor;p=544.96,7892.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1069.02,7921.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1555.23,7818.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2446.71,7891.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3671.23,7964.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4213.02,7930.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4589.92,7755.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5048.18,7994.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5484.17,7841.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5979.03,7998.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1974.21,7580.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3699.86,7595.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4061.82,7606.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5037.46,7615.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=108.89,7421.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=784.40,7450.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1213.25,7302.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2281.27,7294.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2850.72,7421.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5507.32,7261.69");
    }

}

