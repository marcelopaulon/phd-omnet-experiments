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
        @display("i=misc/sensor,p=65.60,183.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=20.75,94.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=167.21,176.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=113.18,11.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=162.06,90.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=249.49,94.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=332.67,144.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=343.24,64.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=199.38,30.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=396.97,251.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=411.51,160.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=517.05,207.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=470.48,132.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=507.53,75.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=449.02,21.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=421.23,78.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=545.12,331.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=622.39,345.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=655.05,413.95");
    }

}

