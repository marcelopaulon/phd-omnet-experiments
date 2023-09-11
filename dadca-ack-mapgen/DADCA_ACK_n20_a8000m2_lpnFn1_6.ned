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

network DADCA_ACK_n20_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=50.93,164.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=12.45,116.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=104.16,257.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=165.08,321.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=223.47,281.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=320.37,372.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=341.65,297.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=426.74,344.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=481.07,400.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=518.40,334.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=590.30,353.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=416.76,451.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=446.73,531.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=298.61,478.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=356.03,517.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=389.44,593.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=304.69,613.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=509.68,520.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=186.45,401.24");
    }

}

