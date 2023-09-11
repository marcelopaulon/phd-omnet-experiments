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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=165.40,128.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=175.30,197.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=107.89,191.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=154.49,304.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=124.98,251.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=276.93,282.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=0.01,219.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=35.11,273.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=41.12,366.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=70.52,473.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=14.03,515.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=178.24,389.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=237.32,188.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=254.27,102.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=149.58,17.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=77.75,13.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=37.12,118.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=314.24,189.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=393.09,133.21");
    }

}

