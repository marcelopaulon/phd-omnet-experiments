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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=12.57,122.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=14.32,189.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=46.41,250.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=128.25,211.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=161.67,272.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=207.25,199.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=266.15,224.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=224.10,287.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=36.94,317.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=92.49,354.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=34.28,384.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=66.71,437.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=122.50,414.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=104.37,294.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=192.07,377.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=82.10,510.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=85.46,576.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=111.01,662.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=29.87,613.03");
    }

}

