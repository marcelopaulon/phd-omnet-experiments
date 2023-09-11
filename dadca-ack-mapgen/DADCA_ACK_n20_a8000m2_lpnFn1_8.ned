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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=80.41,15.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=172.82,8.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=239.93,34.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=245.92,102.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=317.53,182.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=275.80,258.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=403.14,71.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=340.94,21.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=184.73,131.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=321.77,94.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=473.07,113.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=464.38,44.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=381.32,129.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=544.41,152.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=471.61,198.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=518.10,267.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=476.73,335.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=383.43,334.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=433.99,389.43");
    }

}

