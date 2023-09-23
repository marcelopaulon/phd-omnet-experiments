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

network DADCA_ACK_n20_u4_a8000m2_3
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
        @display("i=misc/sensor;p=130.92,7848.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=861.85,7761.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1286.04,7820.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2186.46,7779.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2799.76,7806.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4090.61,7812.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5501.05,7982.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=490.46,7680.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1774.61,7741.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3666.67,7530.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4623.24,7604.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5228.75,7631.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1722.09,7311.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2240.95,7420.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2707.93,7379.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3262.38,7413.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4033.43,7316.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=148.41,7149.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1173.25,7136.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2550.60,7053.24");
    }

}

