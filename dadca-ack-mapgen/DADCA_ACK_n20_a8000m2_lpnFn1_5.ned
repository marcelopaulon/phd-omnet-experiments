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
        @display("i=misc/sensor,p=20.95,121.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=47.55,51.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=136.71,43.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=88.05,6.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=158.82,148.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=158.63,222.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=176.90,310.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=273.07,300.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=241.74,379.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=229.40,249.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=62.17,202.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=286.27,226.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=270.43,146.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=115.11,332.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=362.75,152.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=421.77,202.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=430.21,118.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=497.40,172.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=562.09,168.34");
    }

}

