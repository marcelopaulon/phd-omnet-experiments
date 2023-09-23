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

network DADCA_ACK_n50_u16_a8000m2_6
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
        @display("i=misc/sensor;p=620.54,7952.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1106.36,7937.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1791.34,7969.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2554.35,7911.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3155.95,7900.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4363.17,7909.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5664.25,7833.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=44.54,7606.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=806.10,7653.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1302.65,7507.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2011.83,7689.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2764.49,7513.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3365.81,7615.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4054.45,7667.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5995.39,7708.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=437.86,7302.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4030.77,7265.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4520.65,7270.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5016.46,7493.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5786.02,7392.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=986.38,7199.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1523.25,7140.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1910.61,7199.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2399.20,7130.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3251.04,7219.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3643.10,7127.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5391.08,7234.10");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=300.33,6851.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2960.36,6869.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4053.57,6769.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5112.39,6896.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5892.31,6923.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=702.13,6675.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1429.42,6531.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2203.80,6589.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3160.10,6532.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4715.81,6627.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=241.28,6469.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=969.69,6375.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1788.07,6458.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2693.40,6283.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3757.41,6459.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4366.86,6295.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5375.08,6476.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5812.09,6499.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1433.91,6127.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2166.16,6080.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3306.43,6063.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4836.20,6187.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5433.57,6095.62");
    }

}

