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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=50.81,64.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=100.41,163.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=166.22,112.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=211.21,29.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=264.57,113.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=317.25,183.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=360.91,270.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=438.69,281.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=495.78,210.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=474.22,389.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=530.01,355.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=600.04,318.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=553.74,244.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=409.43,340.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=459.37,454.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=413.64,509.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=351.42,567.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=366.18,660.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=291.93,549.88");
    }

}

