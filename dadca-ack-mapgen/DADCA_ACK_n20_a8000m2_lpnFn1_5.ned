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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=163.87,53.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=202.03,137.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=131.15,189.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=49.74,212.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=118.90,255.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=167.97,338.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=259.73,325.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=178.23,273.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=253.79,237.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=320.30,238.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=391.89,308.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=280.63,415.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=193.41,436.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=161.86,513.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=243.49,552.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=229.91,622.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=285.29,666.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=224.48,727.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=294.02,598.04");
    }

}

