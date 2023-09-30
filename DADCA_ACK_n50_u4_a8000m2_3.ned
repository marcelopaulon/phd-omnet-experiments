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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=472.71,7814.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=824.39,7891.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2500.20,7909.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3191.91,7937.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4170.73,7963.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4824.03,7786.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1942.18,7708.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3280.28,7514.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3895.36,7538.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5518.55,7719.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=755.61,7367.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1371.67,7380.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2800.78,7384.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3558.32,7300.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4235.35,7428.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4884.89,7437.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5729.27,7431.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=481.51,7125.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1785.83,7013.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2209.46,7256.14");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3163.20,7062.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4559.61,7229.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5423.95,7041.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5881.43,7079.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2474.75,6772.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2863.76,6777.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3896.18,6917.32");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4242.37,6848.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4797.74,6860.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=172.00,6686.30");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1012.64,6760.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1465.54,6526.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2092.01,6640.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3425.02,6734.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4074.36,6519.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4508.68,6602.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=584.09,6491.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2684.40,6302.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3164.17,6431.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3692.78,6267.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5272.06,6453.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5633.98,6505.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=208.34,6197.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=538.46,6037.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1224.35,6204.00");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1762.28,6266.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2126.37,6197.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4685.14,6085.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5081.52,6071.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1062.45,5781.24");
    }

}

