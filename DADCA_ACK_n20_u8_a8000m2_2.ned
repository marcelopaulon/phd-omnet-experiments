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

network DADCA_ACK_n20_u8_a8000m2_2
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
        @display("i=misc/sensor;p=853.97,7878.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2848.65,7812.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3557.69,7940.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4138.06,7999.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4638.14,7990.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5070.76,7812.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5885.07,7963.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=167.04,7583.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1588.55,7627.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2080.10,7584.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3249.41,7720.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4349.25,7691.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5862.70,7516.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=714.84,7295.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1067.32,7441.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2488.40,7343.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2857.50,7379.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3356.83,7382.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3847.53,7483.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5020.13,7294.99");
    }

}

