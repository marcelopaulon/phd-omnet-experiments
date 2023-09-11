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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=52.87,55.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=134.13,48.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=201.01,45.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=165.51,108.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=91.28,167.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=159.35,232.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=218.25,168.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=10.67,259.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=0.55,335.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=12.27,171.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=106.95,278.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=337.92,90.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=404.80,135.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=390.22,227.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=430.74,275.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=411.12,367.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=393.88,444.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=329.73,475.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=363.50,568.35");
    }

}

