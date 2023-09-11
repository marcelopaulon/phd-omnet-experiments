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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=42.30,159.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=142.68,149.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=170.55,233.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=197.68,300.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=216.18,361.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=131.12,356.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=182.64,468.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=237.91,505.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=121.70,287.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=47.92,251.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=280.31,264.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=254.96,198.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=103.33,221.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=43.88,384.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=299.87,490.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=318.17,421.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=387.90,399.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=344.17,338.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=344.87,218.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=384.23,168.13");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=495.59,116.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=452.04,52.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=569.21,126.23");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=618.22,59.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=539.34,49.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=309.34,162.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=381.63,53.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=297.88,4.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=221.03,32.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=290.66,69.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=138.60,14.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=236.22,129.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=701.39,67.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=713.64,137.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=652.35,148.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=583.27,185.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=776.94,187.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=839.43,193.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=825.05,103.18");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=919.10,70.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=779.54,37.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=706.09,213.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=918.63,138.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=953.71,217.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=939.78,293.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=924.67,351.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=998.09,312.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1020.11,233.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=995.79,379.73");
    }

}

